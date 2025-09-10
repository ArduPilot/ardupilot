#pragma once
// MESSAGE RC_CHANNELS_OVERRIDE PACKING

#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE 70


typedef struct __mavlink_rc_channels_override_t {
 uint16_t chan1_raw; /*< [us] RC channel 1 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.*/
 uint16_t chan2_raw; /*< [us] RC channel 2 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.*/
 uint16_t chan3_raw; /*< [us] RC channel 3 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.*/
 uint16_t chan4_raw; /*< [us] RC channel 4 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.*/
 uint16_t chan5_raw; /*< [us] RC channel 5 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.*/
 uint16_t chan6_raw; /*< [us] RC channel 6 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.*/
 uint16_t chan7_raw; /*< [us] RC channel 7 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.*/
 uint16_t chan8_raw; /*< [us] RC channel 8 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint16_t chan9_raw; /*< [us] RC channel 9 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.*/
 uint16_t chan10_raw; /*< [us] RC channel 10 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.*/
 uint16_t chan11_raw; /*< [us] RC channel 11 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.*/
 uint16_t chan12_raw; /*< [us] RC channel 12 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.*/
 uint16_t chan13_raw; /*< [us] RC channel 13 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.*/
 uint16_t chan14_raw; /*< [us] RC channel 14 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.*/
 uint16_t chan15_raw; /*< [us] RC channel 15 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.*/
 uint16_t chan16_raw; /*< [us] RC channel 16 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.*/
 uint16_t chan17_raw; /*< [us] RC channel 17 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.*/
 uint16_t chan18_raw; /*< [us] RC channel 18 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.*/
} mavlink_rc_channels_override_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN 38
#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN 18
#define MAVLINK_MSG_ID_70_LEN 38
#define MAVLINK_MSG_ID_70_MIN_LEN 18

#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC 124
#define MAVLINK_MSG_ID_70_CRC 124



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RC_CHANNELS_OVERRIDE { \
    70, \
    "RC_CHANNELS_OVERRIDE", \
    20, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_rc_channels_override_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_rc_channels_override_t, target_component) }, \
         { "chan1_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_rc_channels_override_t, chan1_raw) }, \
         { "chan2_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_rc_channels_override_t, chan2_raw) }, \
         { "chan3_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_rc_channels_override_t, chan3_raw) }, \
         { "chan4_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_rc_channels_override_t, chan4_raw) }, \
         { "chan5_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_rc_channels_override_t, chan5_raw) }, \
         { "chan6_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_rc_channels_override_t, chan6_raw) }, \
         { "chan7_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_rc_channels_override_t, chan7_raw) }, \
         { "chan8_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_rc_channels_override_t, chan8_raw) }, \
         { "chan9_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_rc_channels_override_t, chan9_raw) }, \
         { "chan10_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_rc_channels_override_t, chan10_raw) }, \
         { "chan11_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_rc_channels_override_t, chan11_raw) }, \
         { "chan12_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_rc_channels_override_t, chan12_raw) }, \
         { "chan13_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_rc_channels_override_t, chan13_raw) }, \
         { "chan14_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_rc_channels_override_t, chan14_raw) }, \
         { "chan15_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_rc_channels_override_t, chan15_raw) }, \
         { "chan16_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_rc_channels_override_t, chan16_raw) }, \
         { "chan17_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_rc_channels_override_t, chan17_raw) }, \
         { "chan18_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_rc_channels_override_t, chan18_raw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RC_CHANNELS_OVERRIDE { \
    "RC_CHANNELS_OVERRIDE", \
    20, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_rc_channels_override_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_rc_channels_override_t, target_component) }, \
         { "chan1_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_rc_channels_override_t, chan1_raw) }, \
         { "chan2_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_rc_channels_override_t, chan2_raw) }, \
         { "chan3_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_rc_channels_override_t, chan3_raw) }, \
         { "chan4_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_rc_channels_override_t, chan4_raw) }, \
         { "chan5_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_rc_channels_override_t, chan5_raw) }, \
         { "chan6_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_rc_channels_override_t, chan6_raw) }, \
         { "chan7_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_rc_channels_override_t, chan7_raw) }, \
         { "chan8_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_rc_channels_override_t, chan8_raw) }, \
         { "chan9_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_rc_channels_override_t, chan9_raw) }, \
         { "chan10_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_rc_channels_override_t, chan10_raw) }, \
         { "chan11_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_rc_channels_override_t, chan11_raw) }, \
         { "chan12_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_rc_channels_override_t, chan12_raw) }, \
         { "chan13_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_rc_channels_override_t, chan13_raw) }, \
         { "chan14_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_rc_channels_override_t, chan14_raw) }, \
         { "chan15_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_rc_channels_override_t, chan15_raw) }, \
         { "chan16_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_rc_channels_override_t, chan16_raw) }, \
         { "chan17_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_rc_channels_override_t, chan17_raw) }, \
         { "chan18_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_rc_channels_override_t, chan18_raw) }, \
         } \
}
#endif

/**
 * @brief Pack a rc_channels_override message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param chan1_raw [us] RC channel 1 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan2_raw [us] RC channel 2 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan3_raw [us] RC channel 3 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan4_raw [us] RC channel 4 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan5_raw [us] RC channel 5 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan6_raw [us] RC channel 6 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan7_raw [us] RC channel 7 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan8_raw [us] RC channel 8 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan9_raw [us] RC channel 9 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan10_raw [us] RC channel 10 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan11_raw [us] RC channel 11 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan12_raw [us] RC channel 12 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan13_raw [us] RC channel 13 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan14_raw [us] RC channel 14 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan15_raw [us] RC channel 15 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan16_raw [us] RC channel 16 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan17_raw [us] RC channel 17 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan18_raw [us] RC channel 18 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_override_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN];
    _mav_put_uint16_t(buf, 0, chan1_raw);
    _mav_put_uint16_t(buf, 2, chan2_raw);
    _mav_put_uint16_t(buf, 4, chan3_raw);
    _mav_put_uint16_t(buf, 6, chan4_raw);
    _mav_put_uint16_t(buf, 8, chan5_raw);
    _mav_put_uint16_t(buf, 10, chan6_raw);
    _mav_put_uint16_t(buf, 12, chan7_raw);
    _mav_put_uint16_t(buf, 14, chan8_raw);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint16_t(buf, 18, chan9_raw);
    _mav_put_uint16_t(buf, 20, chan10_raw);
    _mav_put_uint16_t(buf, 22, chan11_raw);
    _mav_put_uint16_t(buf, 24, chan12_raw);
    _mav_put_uint16_t(buf, 26, chan13_raw);
    _mav_put_uint16_t(buf, 28, chan14_raw);
    _mav_put_uint16_t(buf, 30, chan15_raw);
    _mav_put_uint16_t(buf, 32, chan16_raw);
    _mav_put_uint16_t(buf, 34, chan17_raw);
    _mav_put_uint16_t(buf, 36, chan18_raw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#else
    mavlink_rc_channels_override_t packet;
    packet.chan1_raw = chan1_raw;
    packet.chan2_raw = chan2_raw;
    packet.chan3_raw = chan3_raw;
    packet.chan4_raw = chan4_raw;
    packet.chan5_raw = chan5_raw;
    packet.chan6_raw = chan6_raw;
    packet.chan7_raw = chan7_raw;
    packet.chan8_raw = chan8_raw;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.chan9_raw = chan9_raw;
    packet.chan10_raw = chan10_raw;
    packet.chan11_raw = chan11_raw;
    packet.chan12_raw = chan12_raw;
    packet.chan13_raw = chan13_raw;
    packet.chan14_raw = chan14_raw;
    packet.chan15_raw = chan15_raw;
    packet.chan16_raw = chan16_raw;
    packet.chan17_raw = chan17_raw;
    packet.chan18_raw = chan18_raw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
}

/**
 * @brief Pack a rc_channels_override message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param chan1_raw [us] RC channel 1 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan2_raw [us] RC channel 2 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan3_raw [us] RC channel 3 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan4_raw [us] RC channel 4 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan5_raw [us] RC channel 5 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan6_raw [us] RC channel 6 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan7_raw [us] RC channel 7 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan8_raw [us] RC channel 8 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan9_raw [us] RC channel 9 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan10_raw [us] RC channel 10 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan11_raw [us] RC channel 11 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan12_raw [us] RC channel 12 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan13_raw [us] RC channel 13 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan14_raw [us] RC channel 14 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan15_raw [us] RC channel 15 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan16_raw [us] RC channel 16 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan17_raw [us] RC channel 17 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan18_raw [us] RC channel 18 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_override_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN];
    _mav_put_uint16_t(buf, 0, chan1_raw);
    _mav_put_uint16_t(buf, 2, chan2_raw);
    _mav_put_uint16_t(buf, 4, chan3_raw);
    _mav_put_uint16_t(buf, 6, chan4_raw);
    _mav_put_uint16_t(buf, 8, chan5_raw);
    _mav_put_uint16_t(buf, 10, chan6_raw);
    _mav_put_uint16_t(buf, 12, chan7_raw);
    _mav_put_uint16_t(buf, 14, chan8_raw);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint16_t(buf, 18, chan9_raw);
    _mav_put_uint16_t(buf, 20, chan10_raw);
    _mav_put_uint16_t(buf, 22, chan11_raw);
    _mav_put_uint16_t(buf, 24, chan12_raw);
    _mav_put_uint16_t(buf, 26, chan13_raw);
    _mav_put_uint16_t(buf, 28, chan14_raw);
    _mav_put_uint16_t(buf, 30, chan15_raw);
    _mav_put_uint16_t(buf, 32, chan16_raw);
    _mav_put_uint16_t(buf, 34, chan17_raw);
    _mav_put_uint16_t(buf, 36, chan18_raw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#else
    mavlink_rc_channels_override_t packet;
    packet.chan1_raw = chan1_raw;
    packet.chan2_raw = chan2_raw;
    packet.chan3_raw = chan3_raw;
    packet.chan4_raw = chan4_raw;
    packet.chan5_raw = chan5_raw;
    packet.chan6_raw = chan6_raw;
    packet.chan7_raw = chan7_raw;
    packet.chan8_raw = chan8_raw;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.chan9_raw = chan9_raw;
    packet.chan10_raw = chan10_raw;
    packet.chan11_raw = chan11_raw;
    packet.chan12_raw = chan12_raw;
    packet.chan13_raw = chan13_raw;
    packet.chan14_raw = chan14_raw;
    packet.chan15_raw = chan15_raw;
    packet.chan16_raw = chan16_raw;
    packet.chan17_raw = chan17_raw;
    packet.chan18_raw = chan18_raw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#endif
}

/**
 * @brief Pack a rc_channels_override message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param chan1_raw [us] RC channel 1 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan2_raw [us] RC channel 2 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan3_raw [us] RC channel 3 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan4_raw [us] RC channel 4 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan5_raw [us] RC channel 5 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan6_raw [us] RC channel 6 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan7_raw [us] RC channel 7 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan8_raw [us] RC channel 8 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan9_raw [us] RC channel 9 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan10_raw [us] RC channel 10 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan11_raw [us] RC channel 11 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan12_raw [us] RC channel 12 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan13_raw [us] RC channel 13 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan14_raw [us] RC channel 14 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan15_raw [us] RC channel 15 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan16_raw [us] RC channel 16 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan17_raw [us] RC channel 17 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan18_raw [us] RC channel 18 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_override_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t chan1_raw,uint16_t chan2_raw,uint16_t chan3_raw,uint16_t chan4_raw,uint16_t chan5_raw,uint16_t chan6_raw,uint16_t chan7_raw,uint16_t chan8_raw,uint16_t chan9_raw,uint16_t chan10_raw,uint16_t chan11_raw,uint16_t chan12_raw,uint16_t chan13_raw,uint16_t chan14_raw,uint16_t chan15_raw,uint16_t chan16_raw,uint16_t chan17_raw,uint16_t chan18_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN];
    _mav_put_uint16_t(buf, 0, chan1_raw);
    _mav_put_uint16_t(buf, 2, chan2_raw);
    _mav_put_uint16_t(buf, 4, chan3_raw);
    _mav_put_uint16_t(buf, 6, chan4_raw);
    _mav_put_uint16_t(buf, 8, chan5_raw);
    _mav_put_uint16_t(buf, 10, chan6_raw);
    _mav_put_uint16_t(buf, 12, chan7_raw);
    _mav_put_uint16_t(buf, 14, chan8_raw);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint16_t(buf, 18, chan9_raw);
    _mav_put_uint16_t(buf, 20, chan10_raw);
    _mav_put_uint16_t(buf, 22, chan11_raw);
    _mav_put_uint16_t(buf, 24, chan12_raw);
    _mav_put_uint16_t(buf, 26, chan13_raw);
    _mav_put_uint16_t(buf, 28, chan14_raw);
    _mav_put_uint16_t(buf, 30, chan15_raw);
    _mav_put_uint16_t(buf, 32, chan16_raw);
    _mav_put_uint16_t(buf, 34, chan17_raw);
    _mav_put_uint16_t(buf, 36, chan18_raw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#else
    mavlink_rc_channels_override_t packet;
    packet.chan1_raw = chan1_raw;
    packet.chan2_raw = chan2_raw;
    packet.chan3_raw = chan3_raw;
    packet.chan4_raw = chan4_raw;
    packet.chan5_raw = chan5_raw;
    packet.chan6_raw = chan6_raw;
    packet.chan7_raw = chan7_raw;
    packet.chan8_raw = chan8_raw;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.chan9_raw = chan9_raw;
    packet.chan10_raw = chan10_raw;
    packet.chan11_raw = chan11_raw;
    packet.chan12_raw = chan12_raw;
    packet.chan13_raw = chan13_raw;
    packet.chan14_raw = chan14_raw;
    packet.chan15_raw = chan15_raw;
    packet.chan16_raw = chan16_raw;
    packet.chan17_raw = chan17_raw;
    packet.chan18_raw = chan18_raw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
}

/**
 * @brief Encode a rc_channels_override struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_override C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_override_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_override_t* rc_channels_override)
{
    return mavlink_msg_rc_channels_override_pack(system_id, component_id, msg, rc_channels_override->target_system, rc_channels_override->target_component, rc_channels_override->chan1_raw, rc_channels_override->chan2_raw, rc_channels_override->chan3_raw, rc_channels_override->chan4_raw, rc_channels_override->chan5_raw, rc_channels_override->chan6_raw, rc_channels_override->chan7_raw, rc_channels_override->chan8_raw, rc_channels_override->chan9_raw, rc_channels_override->chan10_raw, rc_channels_override->chan11_raw, rc_channels_override->chan12_raw, rc_channels_override->chan13_raw, rc_channels_override->chan14_raw, rc_channels_override->chan15_raw, rc_channels_override->chan16_raw, rc_channels_override->chan17_raw, rc_channels_override->chan18_raw);
}

/**
 * @brief Encode a rc_channels_override struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_override C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_override_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rc_channels_override_t* rc_channels_override)
{
    return mavlink_msg_rc_channels_override_pack_chan(system_id, component_id, chan, msg, rc_channels_override->target_system, rc_channels_override->target_component, rc_channels_override->chan1_raw, rc_channels_override->chan2_raw, rc_channels_override->chan3_raw, rc_channels_override->chan4_raw, rc_channels_override->chan5_raw, rc_channels_override->chan6_raw, rc_channels_override->chan7_raw, rc_channels_override->chan8_raw, rc_channels_override->chan9_raw, rc_channels_override->chan10_raw, rc_channels_override->chan11_raw, rc_channels_override->chan12_raw, rc_channels_override->chan13_raw, rc_channels_override->chan14_raw, rc_channels_override->chan15_raw, rc_channels_override->chan16_raw, rc_channels_override->chan17_raw, rc_channels_override->chan18_raw);
}

/**
 * @brief Encode a rc_channels_override struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_override C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_override_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_rc_channels_override_t* rc_channels_override)
{
    return mavlink_msg_rc_channels_override_pack_status(system_id, component_id, _status, msg,  rc_channels_override->target_system, rc_channels_override->target_component, rc_channels_override->chan1_raw, rc_channels_override->chan2_raw, rc_channels_override->chan3_raw, rc_channels_override->chan4_raw, rc_channels_override->chan5_raw, rc_channels_override->chan6_raw, rc_channels_override->chan7_raw, rc_channels_override->chan8_raw, rc_channels_override->chan9_raw, rc_channels_override->chan10_raw, rc_channels_override->chan11_raw, rc_channels_override->chan12_raw, rc_channels_override->chan13_raw, rc_channels_override->chan14_raw, rc_channels_override->chan15_raw, rc_channels_override->chan16_raw, rc_channels_override->chan17_raw, rc_channels_override->chan18_raw);
}

/**
 * @brief Send a rc_channels_override message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param chan1_raw [us] RC channel 1 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan2_raw [us] RC channel 2 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan3_raw [us] RC channel 3 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan4_raw [us] RC channel 4 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan5_raw [us] RC channel 5 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan6_raw [us] RC channel 6 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan7_raw [us] RC channel 7 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan8_raw [us] RC channel 8 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 * @param chan9_raw [us] RC channel 9 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan10_raw [us] RC channel 10 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan11_raw [us] RC channel 11 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan12_raw [us] RC channel 12 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan13_raw [us] RC channel 13 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan14_raw [us] RC channel 14 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan15_raw [us] RC channel 15 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan16_raw [us] RC channel 16 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan17_raw [us] RC channel 17 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 * @param chan18_raw [us] RC channel 18 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_override_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN];
    _mav_put_uint16_t(buf, 0, chan1_raw);
    _mav_put_uint16_t(buf, 2, chan2_raw);
    _mav_put_uint16_t(buf, 4, chan3_raw);
    _mav_put_uint16_t(buf, 6, chan4_raw);
    _mav_put_uint16_t(buf, 8, chan5_raw);
    _mav_put_uint16_t(buf, 10, chan6_raw);
    _mav_put_uint16_t(buf, 12, chan7_raw);
    _mav_put_uint16_t(buf, 14, chan8_raw);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint16_t(buf, 18, chan9_raw);
    _mav_put_uint16_t(buf, 20, chan10_raw);
    _mav_put_uint16_t(buf, 22, chan11_raw);
    _mav_put_uint16_t(buf, 24, chan12_raw);
    _mav_put_uint16_t(buf, 26, chan13_raw);
    _mav_put_uint16_t(buf, 28, chan14_raw);
    _mav_put_uint16_t(buf, 30, chan15_raw);
    _mav_put_uint16_t(buf, 32, chan16_raw);
    _mav_put_uint16_t(buf, 34, chan17_raw);
    _mav_put_uint16_t(buf, 36, chan18_raw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, buf, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
#else
    mavlink_rc_channels_override_t packet;
    packet.chan1_raw = chan1_raw;
    packet.chan2_raw = chan2_raw;
    packet.chan3_raw = chan3_raw;
    packet.chan4_raw = chan4_raw;
    packet.chan5_raw = chan5_raw;
    packet.chan6_raw = chan6_raw;
    packet.chan7_raw = chan7_raw;
    packet.chan8_raw = chan8_raw;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.chan9_raw = chan9_raw;
    packet.chan10_raw = chan10_raw;
    packet.chan11_raw = chan11_raw;
    packet.chan12_raw = chan12_raw;
    packet.chan13_raw = chan13_raw;
    packet.chan14_raw = chan14_raw;
    packet.chan15_raw = chan15_raw;
    packet.chan16_raw = chan16_raw;
    packet.chan17_raw = chan17_raw;
    packet.chan18_raw = chan18_raw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, (const char *)&packet, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
#endif
}

/**
 * @brief Send a rc_channels_override message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rc_channels_override_send_struct(mavlink_channel_t chan, const mavlink_rc_channels_override_t* rc_channels_override)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rc_channels_override_send(chan, rc_channels_override->target_system, rc_channels_override->target_component, rc_channels_override->chan1_raw, rc_channels_override->chan2_raw, rc_channels_override->chan3_raw, rc_channels_override->chan4_raw, rc_channels_override->chan5_raw, rc_channels_override->chan6_raw, rc_channels_override->chan7_raw, rc_channels_override->chan8_raw, rc_channels_override->chan9_raw, rc_channels_override->chan10_raw, rc_channels_override->chan11_raw, rc_channels_override->chan12_raw, rc_channels_override->chan13_raw, rc_channels_override->chan14_raw, rc_channels_override->chan15_raw, rc_channels_override->chan16_raw, rc_channels_override->chan17_raw, rc_channels_override->chan18_raw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, (const char *)rc_channels_override, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
#endif
}

#if MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rc_channels_override_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, chan1_raw);
    _mav_put_uint16_t(buf, 2, chan2_raw);
    _mav_put_uint16_t(buf, 4, chan3_raw);
    _mav_put_uint16_t(buf, 6, chan4_raw);
    _mav_put_uint16_t(buf, 8, chan5_raw);
    _mav_put_uint16_t(buf, 10, chan6_raw);
    _mav_put_uint16_t(buf, 12, chan7_raw);
    _mav_put_uint16_t(buf, 14, chan8_raw);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint16_t(buf, 18, chan9_raw);
    _mav_put_uint16_t(buf, 20, chan10_raw);
    _mav_put_uint16_t(buf, 22, chan11_raw);
    _mav_put_uint16_t(buf, 24, chan12_raw);
    _mav_put_uint16_t(buf, 26, chan13_raw);
    _mav_put_uint16_t(buf, 28, chan14_raw);
    _mav_put_uint16_t(buf, 30, chan15_raw);
    _mav_put_uint16_t(buf, 32, chan16_raw);
    _mav_put_uint16_t(buf, 34, chan17_raw);
    _mav_put_uint16_t(buf, 36, chan18_raw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, buf, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
#else
    mavlink_rc_channels_override_t *packet = (mavlink_rc_channels_override_t *)msgbuf;
    packet->chan1_raw = chan1_raw;
    packet->chan2_raw = chan2_raw;
    packet->chan3_raw = chan3_raw;
    packet->chan4_raw = chan4_raw;
    packet->chan5_raw = chan5_raw;
    packet->chan6_raw = chan6_raw;
    packet->chan7_raw = chan7_raw;
    packet->chan8_raw = chan8_raw;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->chan9_raw = chan9_raw;
    packet->chan10_raw = chan10_raw;
    packet->chan11_raw = chan11_raw;
    packet->chan12_raw = chan12_raw;
    packet->chan13_raw = chan13_raw;
    packet->chan14_raw = chan14_raw;
    packet->chan15_raw = chan15_raw;
    packet->chan16_raw = chan16_raw;
    packet->chan17_raw = chan17_raw;
    packet->chan18_raw = chan18_raw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, (const char *)packet, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
#endif
}
#endif

#endif

// MESSAGE RC_CHANNELS_OVERRIDE UNPACKING


/**
 * @brief Get field target_system from rc_channels_override message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_rc_channels_override_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_component from rc_channels_override message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_rc_channels_override_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field chan1_raw from rc_channels_override message
 *
 * @return [us] RC channel 1 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan1_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field chan2_raw from rc_channels_override message
 *
 * @return [us] RC channel 2 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan2_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field chan3_raw from rc_channels_override message
 *
 * @return [us] RC channel 3 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan3_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field chan4_raw from rc_channels_override message
 *
 * @return [us] RC channel 4 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan4_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field chan5_raw from rc_channels_override message
 *
 * @return [us] RC channel 5 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan5_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field chan6_raw from rc_channels_override message
 *
 * @return [us] RC channel 6 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan6_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field chan7_raw from rc_channels_override message
 *
 * @return [us] RC channel 7 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan7_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field chan8_raw from rc_channels_override message
 *
 * @return [us] RC channel 8 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan8_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field chan9_raw from rc_channels_override message
 *
 * @return [us] RC channel 9 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan9_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field chan10_raw from rc_channels_override message
 *
 * @return [us] RC channel 10 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan10_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field chan11_raw from rc_channels_override message
 *
 * @return [us] RC channel 11 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan11_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field chan12_raw from rc_channels_override message
 *
 * @return [us] RC channel 12 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan12_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field chan13_raw from rc_channels_override message
 *
 * @return [us] RC channel 13 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan13_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field chan14_raw from rc_channels_override message
 *
 * @return [us] RC channel 14 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan14_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field chan15_raw from rc_channels_override message
 *
 * @return [us] RC channel 15 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan15_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field chan16_raw from rc_channels_override message
 *
 * @return [us] RC channel 16 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan16_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field chan17_raw from rc_channels_override message
 *
 * @return [us] RC channel 17 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan17_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field chan18_raw from rc_channels_override message
 *
 * @return [us] RC channel 18 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan18_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Decode a rc_channels_override message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_override C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_override_decode(const mavlink_message_t* msg, mavlink_rc_channels_override_t* rc_channels_override)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rc_channels_override->chan1_raw = mavlink_msg_rc_channels_override_get_chan1_raw(msg);
    rc_channels_override->chan2_raw = mavlink_msg_rc_channels_override_get_chan2_raw(msg);
    rc_channels_override->chan3_raw = mavlink_msg_rc_channels_override_get_chan3_raw(msg);
    rc_channels_override->chan4_raw = mavlink_msg_rc_channels_override_get_chan4_raw(msg);
    rc_channels_override->chan5_raw = mavlink_msg_rc_channels_override_get_chan5_raw(msg);
    rc_channels_override->chan6_raw = mavlink_msg_rc_channels_override_get_chan6_raw(msg);
    rc_channels_override->chan7_raw = mavlink_msg_rc_channels_override_get_chan7_raw(msg);
    rc_channels_override->chan8_raw = mavlink_msg_rc_channels_override_get_chan8_raw(msg);
    rc_channels_override->target_system = mavlink_msg_rc_channels_override_get_target_system(msg);
    rc_channels_override->target_component = mavlink_msg_rc_channels_override_get_target_component(msg);
    rc_channels_override->chan9_raw = mavlink_msg_rc_channels_override_get_chan9_raw(msg);
    rc_channels_override->chan10_raw = mavlink_msg_rc_channels_override_get_chan10_raw(msg);
    rc_channels_override->chan11_raw = mavlink_msg_rc_channels_override_get_chan11_raw(msg);
    rc_channels_override->chan12_raw = mavlink_msg_rc_channels_override_get_chan12_raw(msg);
    rc_channels_override->chan13_raw = mavlink_msg_rc_channels_override_get_chan13_raw(msg);
    rc_channels_override->chan14_raw = mavlink_msg_rc_channels_override_get_chan14_raw(msg);
    rc_channels_override->chan15_raw = mavlink_msg_rc_channels_override_get_chan15_raw(msg);
    rc_channels_override->chan16_raw = mavlink_msg_rc_channels_override_get_chan16_raw(msg);
    rc_channels_override->chan17_raw = mavlink_msg_rc_channels_override_get_chan17_raw(msg);
    rc_channels_override->chan18_raw = mavlink_msg_rc_channels_override_get_chan18_raw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN? msg->len : MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN;
        memset(rc_channels_override, 0, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
    memcpy(rc_channels_override, _MAV_PAYLOAD(msg), len);
#endif
}
