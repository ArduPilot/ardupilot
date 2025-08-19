#pragma once
// MESSAGE SENS_POWER_BOARD PACKING

#define MAVLINK_MSG_ID_SENS_POWER_BOARD 8013


typedef struct __mavlink_sens_power_board_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 float pwr_brd_system_volt; /*< [V] Power board system voltage*/
 float pwr_brd_servo_volt; /*< [V] Power board servo voltage*/
 float pwr_brd_digital_volt; /*< [V] Power board digital voltage*/
 float pwr_brd_mot_l_amp; /*< [A] Power board left motor current sensor*/
 float pwr_brd_mot_r_amp; /*< [A] Power board right motor current sensor*/
 float pwr_brd_analog_amp; /*< [A] Power board analog current sensor*/
 float pwr_brd_digital_amp; /*< [A] Power board digital current sensor*/
 float pwr_brd_ext_amp; /*< [A] Power board extension current sensor*/
 float pwr_brd_aux_amp; /*< [A] Power board aux current sensor*/
 uint8_t pwr_brd_status; /*<  Power board status register*/
 uint8_t pwr_brd_led_status; /*<  Power board leds status*/
} mavlink_sens_power_board_t;

#define MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN 46
#define MAVLINK_MSG_ID_SENS_POWER_BOARD_MIN_LEN 46
#define MAVLINK_MSG_ID_8013_LEN 46
#define MAVLINK_MSG_ID_8013_MIN_LEN 46

#define MAVLINK_MSG_ID_SENS_POWER_BOARD_CRC 222
#define MAVLINK_MSG_ID_8013_CRC 222



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENS_POWER_BOARD { \
    8013, \
    "SENS_POWER_BOARD", \
    12, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sens_power_board_t, timestamp) }, \
         { "pwr_brd_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_sens_power_board_t, pwr_brd_status) }, \
         { "pwr_brd_led_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_sens_power_board_t, pwr_brd_led_status) }, \
         { "pwr_brd_system_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sens_power_board_t, pwr_brd_system_volt) }, \
         { "pwr_brd_servo_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sens_power_board_t, pwr_brd_servo_volt) }, \
         { "pwr_brd_digital_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sens_power_board_t, pwr_brd_digital_volt) }, \
         { "pwr_brd_mot_l_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sens_power_board_t, pwr_brd_mot_l_amp) }, \
         { "pwr_brd_mot_r_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sens_power_board_t, pwr_brd_mot_r_amp) }, \
         { "pwr_brd_analog_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sens_power_board_t, pwr_brd_analog_amp) }, \
         { "pwr_brd_digital_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sens_power_board_t, pwr_brd_digital_amp) }, \
         { "pwr_brd_ext_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sens_power_board_t, pwr_brd_ext_amp) }, \
         { "pwr_brd_aux_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sens_power_board_t, pwr_brd_aux_amp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENS_POWER_BOARD { \
    "SENS_POWER_BOARD", \
    12, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sens_power_board_t, timestamp) }, \
         { "pwr_brd_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_sens_power_board_t, pwr_brd_status) }, \
         { "pwr_brd_led_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_sens_power_board_t, pwr_brd_led_status) }, \
         { "pwr_brd_system_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sens_power_board_t, pwr_brd_system_volt) }, \
         { "pwr_brd_servo_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sens_power_board_t, pwr_brd_servo_volt) }, \
         { "pwr_brd_digital_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sens_power_board_t, pwr_brd_digital_volt) }, \
         { "pwr_brd_mot_l_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sens_power_board_t, pwr_brd_mot_l_amp) }, \
         { "pwr_brd_mot_r_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sens_power_board_t, pwr_brd_mot_r_amp) }, \
         { "pwr_brd_analog_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sens_power_board_t, pwr_brd_analog_amp) }, \
         { "pwr_brd_digital_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sens_power_board_t, pwr_brd_digital_amp) }, \
         { "pwr_brd_ext_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sens_power_board_t, pwr_brd_ext_amp) }, \
         { "pwr_brd_aux_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sens_power_board_t, pwr_brd_aux_amp) }, \
         } \
}
#endif

/**
 * @brief Pack a sens_power_board message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param pwr_brd_status  Power board status register
 * @param pwr_brd_led_status  Power board leds status
 * @param pwr_brd_system_volt [V] Power board system voltage
 * @param pwr_brd_servo_volt [V] Power board servo voltage
 * @param pwr_brd_digital_volt [V] Power board digital voltage
 * @param pwr_brd_mot_l_amp [A] Power board left motor current sensor
 * @param pwr_brd_mot_r_amp [A] Power board right motor current sensor
 * @param pwr_brd_analog_amp [A] Power board analog current sensor
 * @param pwr_brd_digital_amp [A] Power board digital current sensor
 * @param pwr_brd_ext_amp [A] Power board extension current sensor
 * @param pwr_brd_aux_amp [A] Power board aux current sensor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_power_board_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t pwr_brd_status, uint8_t pwr_brd_led_status, float pwr_brd_system_volt, float pwr_brd_servo_volt, float pwr_brd_digital_volt, float pwr_brd_mot_l_amp, float pwr_brd_mot_r_amp, float pwr_brd_analog_amp, float pwr_brd_digital_amp, float pwr_brd_ext_amp, float pwr_brd_aux_amp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pwr_brd_system_volt);
    _mav_put_float(buf, 12, pwr_brd_servo_volt);
    _mav_put_float(buf, 16, pwr_brd_digital_volt);
    _mav_put_float(buf, 20, pwr_brd_mot_l_amp);
    _mav_put_float(buf, 24, pwr_brd_mot_r_amp);
    _mav_put_float(buf, 28, pwr_brd_analog_amp);
    _mav_put_float(buf, 32, pwr_brd_digital_amp);
    _mav_put_float(buf, 36, pwr_brd_ext_amp);
    _mav_put_float(buf, 40, pwr_brd_aux_amp);
    _mav_put_uint8_t(buf, 44, pwr_brd_status);
    _mav_put_uint8_t(buf, 45, pwr_brd_led_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN);
#else
    mavlink_sens_power_board_t packet;
    packet.timestamp = timestamp;
    packet.pwr_brd_system_volt = pwr_brd_system_volt;
    packet.pwr_brd_servo_volt = pwr_brd_servo_volt;
    packet.pwr_brd_digital_volt = pwr_brd_digital_volt;
    packet.pwr_brd_mot_l_amp = pwr_brd_mot_l_amp;
    packet.pwr_brd_mot_r_amp = pwr_brd_mot_r_amp;
    packet.pwr_brd_analog_amp = pwr_brd_analog_amp;
    packet.pwr_brd_digital_amp = pwr_brd_digital_amp;
    packet.pwr_brd_ext_amp = pwr_brd_ext_amp;
    packet.pwr_brd_aux_amp = pwr_brd_aux_amp;
    packet.pwr_brd_status = pwr_brd_status;
    packet.pwr_brd_led_status = pwr_brd_led_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_POWER_BOARD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENS_POWER_BOARD_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_CRC);
}

/**
 * @brief Pack a sens_power_board message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param pwr_brd_status  Power board status register
 * @param pwr_brd_led_status  Power board leds status
 * @param pwr_brd_system_volt [V] Power board system voltage
 * @param pwr_brd_servo_volt [V] Power board servo voltage
 * @param pwr_brd_digital_volt [V] Power board digital voltage
 * @param pwr_brd_mot_l_amp [A] Power board left motor current sensor
 * @param pwr_brd_mot_r_amp [A] Power board right motor current sensor
 * @param pwr_brd_analog_amp [A] Power board analog current sensor
 * @param pwr_brd_digital_amp [A] Power board digital current sensor
 * @param pwr_brd_ext_amp [A] Power board extension current sensor
 * @param pwr_brd_aux_amp [A] Power board aux current sensor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_power_board_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t pwr_brd_status, uint8_t pwr_brd_led_status, float pwr_brd_system_volt, float pwr_brd_servo_volt, float pwr_brd_digital_volt, float pwr_brd_mot_l_amp, float pwr_brd_mot_r_amp, float pwr_brd_analog_amp, float pwr_brd_digital_amp, float pwr_brd_ext_amp, float pwr_brd_aux_amp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pwr_brd_system_volt);
    _mav_put_float(buf, 12, pwr_brd_servo_volt);
    _mav_put_float(buf, 16, pwr_brd_digital_volt);
    _mav_put_float(buf, 20, pwr_brd_mot_l_amp);
    _mav_put_float(buf, 24, pwr_brd_mot_r_amp);
    _mav_put_float(buf, 28, pwr_brd_analog_amp);
    _mav_put_float(buf, 32, pwr_brd_digital_amp);
    _mav_put_float(buf, 36, pwr_brd_ext_amp);
    _mav_put_float(buf, 40, pwr_brd_aux_amp);
    _mav_put_uint8_t(buf, 44, pwr_brd_status);
    _mav_put_uint8_t(buf, 45, pwr_brd_led_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN);
#else
    mavlink_sens_power_board_t packet;
    packet.timestamp = timestamp;
    packet.pwr_brd_system_volt = pwr_brd_system_volt;
    packet.pwr_brd_servo_volt = pwr_brd_servo_volt;
    packet.pwr_brd_digital_volt = pwr_brd_digital_volt;
    packet.pwr_brd_mot_l_amp = pwr_brd_mot_l_amp;
    packet.pwr_brd_mot_r_amp = pwr_brd_mot_r_amp;
    packet.pwr_brd_analog_amp = pwr_brd_analog_amp;
    packet.pwr_brd_digital_amp = pwr_brd_digital_amp;
    packet.pwr_brd_ext_amp = pwr_brd_ext_amp;
    packet.pwr_brd_aux_amp = pwr_brd_aux_amp;
    packet.pwr_brd_status = pwr_brd_status;
    packet.pwr_brd_led_status = pwr_brd_led_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_POWER_BOARD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SENS_POWER_BOARD_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SENS_POWER_BOARD_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN);
#endif
}

/**
 * @brief Pack a sens_power_board message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param pwr_brd_status  Power board status register
 * @param pwr_brd_led_status  Power board leds status
 * @param pwr_brd_system_volt [V] Power board system voltage
 * @param pwr_brd_servo_volt [V] Power board servo voltage
 * @param pwr_brd_digital_volt [V] Power board digital voltage
 * @param pwr_brd_mot_l_amp [A] Power board left motor current sensor
 * @param pwr_brd_mot_r_amp [A] Power board right motor current sensor
 * @param pwr_brd_analog_amp [A] Power board analog current sensor
 * @param pwr_brd_digital_amp [A] Power board digital current sensor
 * @param pwr_brd_ext_amp [A] Power board extension current sensor
 * @param pwr_brd_aux_amp [A] Power board aux current sensor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_power_board_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t pwr_brd_status,uint8_t pwr_brd_led_status,float pwr_brd_system_volt,float pwr_brd_servo_volt,float pwr_brd_digital_volt,float pwr_brd_mot_l_amp,float pwr_brd_mot_r_amp,float pwr_brd_analog_amp,float pwr_brd_digital_amp,float pwr_brd_ext_amp,float pwr_brd_aux_amp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pwr_brd_system_volt);
    _mav_put_float(buf, 12, pwr_brd_servo_volt);
    _mav_put_float(buf, 16, pwr_brd_digital_volt);
    _mav_put_float(buf, 20, pwr_brd_mot_l_amp);
    _mav_put_float(buf, 24, pwr_brd_mot_r_amp);
    _mav_put_float(buf, 28, pwr_brd_analog_amp);
    _mav_put_float(buf, 32, pwr_brd_digital_amp);
    _mav_put_float(buf, 36, pwr_brd_ext_amp);
    _mav_put_float(buf, 40, pwr_brd_aux_amp);
    _mav_put_uint8_t(buf, 44, pwr_brd_status);
    _mav_put_uint8_t(buf, 45, pwr_brd_led_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN);
#else
    mavlink_sens_power_board_t packet;
    packet.timestamp = timestamp;
    packet.pwr_brd_system_volt = pwr_brd_system_volt;
    packet.pwr_brd_servo_volt = pwr_brd_servo_volt;
    packet.pwr_brd_digital_volt = pwr_brd_digital_volt;
    packet.pwr_brd_mot_l_amp = pwr_brd_mot_l_amp;
    packet.pwr_brd_mot_r_amp = pwr_brd_mot_r_amp;
    packet.pwr_brd_analog_amp = pwr_brd_analog_amp;
    packet.pwr_brd_digital_amp = pwr_brd_digital_amp;
    packet.pwr_brd_ext_amp = pwr_brd_ext_amp;
    packet.pwr_brd_aux_amp = pwr_brd_aux_amp;
    packet.pwr_brd_status = pwr_brd_status;
    packet.pwr_brd_led_status = pwr_brd_led_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_POWER_BOARD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENS_POWER_BOARD_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_CRC);
}

/**
 * @brief Encode a sens_power_board struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sens_power_board C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_power_board_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sens_power_board_t* sens_power_board)
{
    return mavlink_msg_sens_power_board_pack(system_id, component_id, msg, sens_power_board->timestamp, sens_power_board->pwr_brd_status, sens_power_board->pwr_brd_led_status, sens_power_board->pwr_brd_system_volt, sens_power_board->pwr_brd_servo_volt, sens_power_board->pwr_brd_digital_volt, sens_power_board->pwr_brd_mot_l_amp, sens_power_board->pwr_brd_mot_r_amp, sens_power_board->pwr_brd_analog_amp, sens_power_board->pwr_brd_digital_amp, sens_power_board->pwr_brd_ext_amp, sens_power_board->pwr_brd_aux_amp);
}

/**
 * @brief Encode a sens_power_board struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sens_power_board C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_power_board_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sens_power_board_t* sens_power_board)
{
    return mavlink_msg_sens_power_board_pack_chan(system_id, component_id, chan, msg, sens_power_board->timestamp, sens_power_board->pwr_brd_status, sens_power_board->pwr_brd_led_status, sens_power_board->pwr_brd_system_volt, sens_power_board->pwr_brd_servo_volt, sens_power_board->pwr_brd_digital_volt, sens_power_board->pwr_brd_mot_l_amp, sens_power_board->pwr_brd_mot_r_amp, sens_power_board->pwr_brd_analog_amp, sens_power_board->pwr_brd_digital_amp, sens_power_board->pwr_brd_ext_amp, sens_power_board->pwr_brd_aux_amp);
}

/**
 * @brief Encode a sens_power_board struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param sens_power_board C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_power_board_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_sens_power_board_t* sens_power_board)
{
    return mavlink_msg_sens_power_board_pack_status(system_id, component_id, _status, msg,  sens_power_board->timestamp, sens_power_board->pwr_brd_status, sens_power_board->pwr_brd_led_status, sens_power_board->pwr_brd_system_volt, sens_power_board->pwr_brd_servo_volt, sens_power_board->pwr_brd_digital_volt, sens_power_board->pwr_brd_mot_l_amp, sens_power_board->pwr_brd_mot_r_amp, sens_power_board->pwr_brd_analog_amp, sens_power_board->pwr_brd_digital_amp, sens_power_board->pwr_brd_ext_amp, sens_power_board->pwr_brd_aux_amp);
}

/**
 * @brief Send a sens_power_board message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param pwr_brd_status  Power board status register
 * @param pwr_brd_led_status  Power board leds status
 * @param pwr_brd_system_volt [V] Power board system voltage
 * @param pwr_brd_servo_volt [V] Power board servo voltage
 * @param pwr_brd_digital_volt [V] Power board digital voltage
 * @param pwr_brd_mot_l_amp [A] Power board left motor current sensor
 * @param pwr_brd_mot_r_amp [A] Power board right motor current sensor
 * @param pwr_brd_analog_amp [A] Power board analog current sensor
 * @param pwr_brd_digital_amp [A] Power board digital current sensor
 * @param pwr_brd_ext_amp [A] Power board extension current sensor
 * @param pwr_brd_aux_amp [A] Power board aux current sensor
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sens_power_board_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t pwr_brd_status, uint8_t pwr_brd_led_status, float pwr_brd_system_volt, float pwr_brd_servo_volt, float pwr_brd_digital_volt, float pwr_brd_mot_l_amp, float pwr_brd_mot_r_amp, float pwr_brd_analog_amp, float pwr_brd_digital_amp, float pwr_brd_ext_amp, float pwr_brd_aux_amp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pwr_brd_system_volt);
    _mav_put_float(buf, 12, pwr_brd_servo_volt);
    _mav_put_float(buf, 16, pwr_brd_digital_volt);
    _mav_put_float(buf, 20, pwr_brd_mot_l_amp);
    _mav_put_float(buf, 24, pwr_brd_mot_r_amp);
    _mav_put_float(buf, 28, pwr_brd_analog_amp);
    _mav_put_float(buf, 32, pwr_brd_digital_amp);
    _mav_put_float(buf, 36, pwr_brd_ext_amp);
    _mav_put_float(buf, 40, pwr_brd_aux_amp);
    _mav_put_uint8_t(buf, 44, pwr_brd_status);
    _mav_put_uint8_t(buf, 45, pwr_brd_led_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_POWER_BOARD, buf, MAVLINK_MSG_ID_SENS_POWER_BOARD_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_CRC);
#else
    mavlink_sens_power_board_t packet;
    packet.timestamp = timestamp;
    packet.pwr_brd_system_volt = pwr_brd_system_volt;
    packet.pwr_brd_servo_volt = pwr_brd_servo_volt;
    packet.pwr_brd_digital_volt = pwr_brd_digital_volt;
    packet.pwr_brd_mot_l_amp = pwr_brd_mot_l_amp;
    packet.pwr_brd_mot_r_amp = pwr_brd_mot_r_amp;
    packet.pwr_brd_analog_amp = pwr_brd_analog_amp;
    packet.pwr_brd_digital_amp = pwr_brd_digital_amp;
    packet.pwr_brd_ext_amp = pwr_brd_ext_amp;
    packet.pwr_brd_aux_amp = pwr_brd_aux_amp;
    packet.pwr_brd_status = pwr_brd_status;
    packet.pwr_brd_led_status = pwr_brd_led_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_POWER_BOARD, (const char *)&packet, MAVLINK_MSG_ID_SENS_POWER_BOARD_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_CRC);
#endif
}

/**
 * @brief Send a sens_power_board message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sens_power_board_send_struct(mavlink_channel_t chan, const mavlink_sens_power_board_t* sens_power_board)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sens_power_board_send(chan, sens_power_board->timestamp, sens_power_board->pwr_brd_status, sens_power_board->pwr_brd_led_status, sens_power_board->pwr_brd_system_volt, sens_power_board->pwr_brd_servo_volt, sens_power_board->pwr_brd_digital_volt, sens_power_board->pwr_brd_mot_l_amp, sens_power_board->pwr_brd_mot_r_amp, sens_power_board->pwr_brd_analog_amp, sens_power_board->pwr_brd_digital_amp, sens_power_board->pwr_brd_ext_amp, sens_power_board->pwr_brd_aux_amp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_POWER_BOARD, (const char *)sens_power_board, MAVLINK_MSG_ID_SENS_POWER_BOARD_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sens_power_board_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t pwr_brd_status, uint8_t pwr_brd_led_status, float pwr_brd_system_volt, float pwr_brd_servo_volt, float pwr_brd_digital_volt, float pwr_brd_mot_l_amp, float pwr_brd_mot_r_amp, float pwr_brd_analog_amp, float pwr_brd_digital_amp, float pwr_brd_ext_amp, float pwr_brd_aux_amp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pwr_brd_system_volt);
    _mav_put_float(buf, 12, pwr_brd_servo_volt);
    _mav_put_float(buf, 16, pwr_brd_digital_volt);
    _mav_put_float(buf, 20, pwr_brd_mot_l_amp);
    _mav_put_float(buf, 24, pwr_brd_mot_r_amp);
    _mav_put_float(buf, 28, pwr_brd_analog_amp);
    _mav_put_float(buf, 32, pwr_brd_digital_amp);
    _mav_put_float(buf, 36, pwr_brd_ext_amp);
    _mav_put_float(buf, 40, pwr_brd_aux_amp);
    _mav_put_uint8_t(buf, 44, pwr_brd_status);
    _mav_put_uint8_t(buf, 45, pwr_brd_led_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_POWER_BOARD, buf, MAVLINK_MSG_ID_SENS_POWER_BOARD_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_CRC);
#else
    mavlink_sens_power_board_t *packet = (mavlink_sens_power_board_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->pwr_brd_system_volt = pwr_brd_system_volt;
    packet->pwr_brd_servo_volt = pwr_brd_servo_volt;
    packet->pwr_brd_digital_volt = pwr_brd_digital_volt;
    packet->pwr_brd_mot_l_amp = pwr_brd_mot_l_amp;
    packet->pwr_brd_mot_r_amp = pwr_brd_mot_r_amp;
    packet->pwr_brd_analog_amp = pwr_brd_analog_amp;
    packet->pwr_brd_digital_amp = pwr_brd_digital_amp;
    packet->pwr_brd_ext_amp = pwr_brd_ext_amp;
    packet->pwr_brd_aux_amp = pwr_brd_aux_amp;
    packet->pwr_brd_status = pwr_brd_status;
    packet->pwr_brd_led_status = pwr_brd_led_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_POWER_BOARD, (const char *)packet, MAVLINK_MSG_ID_SENS_POWER_BOARD_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN, MAVLINK_MSG_ID_SENS_POWER_BOARD_CRC);
#endif
}
#endif

#endif

// MESSAGE SENS_POWER_BOARD UNPACKING


/**
 * @brief Get field timestamp from sens_power_board message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_sens_power_board_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field pwr_brd_status from sens_power_board message
 *
 * @return  Power board status register
 */
static inline uint8_t mavlink_msg_sens_power_board_get_pwr_brd_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field pwr_brd_led_status from sens_power_board message
 *
 * @return  Power board leds status
 */
static inline uint8_t mavlink_msg_sens_power_board_get_pwr_brd_led_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field pwr_brd_system_volt from sens_power_board message
 *
 * @return [V] Power board system voltage
 */
static inline float mavlink_msg_sens_power_board_get_pwr_brd_system_volt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pwr_brd_servo_volt from sens_power_board message
 *
 * @return [V] Power board servo voltage
 */
static inline float mavlink_msg_sens_power_board_get_pwr_brd_servo_volt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pwr_brd_digital_volt from sens_power_board message
 *
 * @return [V] Power board digital voltage
 */
static inline float mavlink_msg_sens_power_board_get_pwr_brd_digital_volt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pwr_brd_mot_l_amp from sens_power_board message
 *
 * @return [A] Power board left motor current sensor
 */
static inline float mavlink_msg_sens_power_board_get_pwr_brd_mot_l_amp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pwr_brd_mot_r_amp from sens_power_board message
 *
 * @return [A] Power board right motor current sensor
 */
static inline float mavlink_msg_sens_power_board_get_pwr_brd_mot_r_amp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field pwr_brd_analog_amp from sens_power_board message
 *
 * @return [A] Power board analog current sensor
 */
static inline float mavlink_msg_sens_power_board_get_pwr_brd_analog_amp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field pwr_brd_digital_amp from sens_power_board message
 *
 * @return [A] Power board digital current sensor
 */
static inline float mavlink_msg_sens_power_board_get_pwr_brd_digital_amp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field pwr_brd_ext_amp from sens_power_board message
 *
 * @return [A] Power board extension current sensor
 */
static inline float mavlink_msg_sens_power_board_get_pwr_brd_ext_amp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field pwr_brd_aux_amp from sens_power_board message
 *
 * @return [A] Power board aux current sensor
 */
static inline float mavlink_msg_sens_power_board_get_pwr_brd_aux_amp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a sens_power_board message into a struct
 *
 * @param msg The message to decode
 * @param sens_power_board C-struct to decode the message contents into
 */
static inline void mavlink_msg_sens_power_board_decode(const mavlink_message_t* msg, mavlink_sens_power_board_t* sens_power_board)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sens_power_board->timestamp = mavlink_msg_sens_power_board_get_timestamp(msg);
    sens_power_board->pwr_brd_system_volt = mavlink_msg_sens_power_board_get_pwr_brd_system_volt(msg);
    sens_power_board->pwr_brd_servo_volt = mavlink_msg_sens_power_board_get_pwr_brd_servo_volt(msg);
    sens_power_board->pwr_brd_digital_volt = mavlink_msg_sens_power_board_get_pwr_brd_digital_volt(msg);
    sens_power_board->pwr_brd_mot_l_amp = mavlink_msg_sens_power_board_get_pwr_brd_mot_l_amp(msg);
    sens_power_board->pwr_brd_mot_r_amp = mavlink_msg_sens_power_board_get_pwr_brd_mot_r_amp(msg);
    sens_power_board->pwr_brd_analog_amp = mavlink_msg_sens_power_board_get_pwr_brd_analog_amp(msg);
    sens_power_board->pwr_brd_digital_amp = mavlink_msg_sens_power_board_get_pwr_brd_digital_amp(msg);
    sens_power_board->pwr_brd_ext_amp = mavlink_msg_sens_power_board_get_pwr_brd_ext_amp(msg);
    sens_power_board->pwr_brd_aux_amp = mavlink_msg_sens_power_board_get_pwr_brd_aux_amp(msg);
    sens_power_board->pwr_brd_status = mavlink_msg_sens_power_board_get_pwr_brd_status(msg);
    sens_power_board->pwr_brd_led_status = mavlink_msg_sens_power_board_get_pwr_brd_led_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN? msg->len : MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN;
        memset(sens_power_board, 0, MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN);
    memcpy(sens_power_board, _MAV_PAYLOAD(msg), len);
#endif
}
